using System.Collections;
using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class LayerChordHelperCacheTests
{
    private static readonly VCarveCutterProfile Cutter=new([
        new(CarvingCutterSectionKind.Hemisphere,0,.4,.4),new(CarvingCutterSectionKind.Linear,.4,2,3.6)],2);
    private static PointD[] Box(double x,double y,double r)=>[new(x-r,y-r),new(x+r,y-r),new(x+r,y+r),new(x-r,y+r)];
    private static readonly (Point3D A,Point3D B) Good=(new(-.3,0,-.2),new(.3,0,-.2));
    private static readonly (Point3D A,Point3D B) Bad=(new(-.3,3.5,-.3),new(.3,3.5,-.3));
    private static VCarveMaterialSurface Owner(int? limit,CancellationToken token=default,Action<string>? log=null)
    {
        var old=VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value;
        try
        {
            VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value=limit;
            var stock=new VCarveClearedStock([new(new(-3,-4,-1),new(3,-4,-1),.4,.4)],[1,2],Box(0,0,10),
                [new(Cutter,[new(new(-2,3.5,-.05),new(2,3.5,-.05),0)])]);
            return new([Box(0,0,2)],stock,Cutter,2,0,token,log);
        }
        finally{VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value=old;}
    }
    private static Func<Point3D,Point3D,bool> Layer(VCarveMaterialSurface owner,bool core=false)
        =>typeof(VCarveMaterialSurface).GetMethod(core?"LayerLegalCore":"LayerLegal",BindingFlags.NonPublic|BindingFlags.Instance)!
            .CreateDelegate<Func<Point3D,Point3D,bool>>(owner);
    private static T Field<T>(object owner,string name)=>(T)owner.GetType().GetField(name,BindingFlags.NonPublic|BindingFlags.Instance)!.GetValue(owner)!;
    private static long Count(VCarveMaterialSurface owner,string name)=>Field<long>(owner,name);
    private static int Entries(VCarveMaterialSurface owner)=>Field<IDictionary?>(owner,"layerChordProofs")?.Count??0;
    private static Dictionary<string,long> StockCounters(VCarveMaterialSurface owner)
    {
        var stock=Field<object>(owner,"projectedStock");
        return stock.GetType().GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance)
            .Where(f=>f.FieldType==typeof(long)).ToDictionary(f=>f.Name,f=>(long)f.GetValue(stock)!);
    }

    [Fact]
    public void DefaultKeepsLiteralHelperAndAllocatesNoChordCacheStorage()
    {
        var owner=Owner(null);var prove=Layer(owner);
        Assert.Equal(0,Field<int>(owner,"layerChordCacheEntries"));
        Assert.Null(Field<IDictionary?>(owner,"layerChordProofs"));Assert.Null(Field<object?>(owner,"layerChordOrder"));
        Assert.True(prove(Good.A,Good.B));Assert.True(prove(Good.A,Good.B));
        Assert.Equal(2,Count(owner,"layerChordCoreRuns"));Assert.Equal(0,Count(owner,"layerChordPositiveHits"));
        var worker=owner.CreateNativeQueryWorker(out _);
        Assert.Equal(0,Field<int>(worker,"layerChordCacheEntries"));Assert.Null(Field<IDictionary?>(worker,"layerChordProofs"));
    }

    [Fact]
    public void PositiveAndNegativeHitsPreserveLiteralHelperAnswersAndFullStockFallbackWork()
    {
        var literal=Owner(0);var cached=Owner(32768);var pure=Layer(Owner(0),core:true);
        Assert.True(pure(Good.A,Good.B));Assert.False(pure(Bad.A,Bad.B));
        var sequence=new[]{Good,Good,Bad,Bad,Bad,(Good.B,Good.A),(Good.B,Good.A),Good};
        foreach(var (a,b) in sequence)
        {
            Assert.Equal(literal.IsLegalNativeChord(a,b),cached.IsLegalNativeChord(a,b));
            var expected=StockCounters(literal);var actual=StockCounters(cached);
            foreach(var pair in expected)Assert.Equal(pair.Value,actual[pair.Key]);
        }
        Assert.True(Count(cached,"layerChordPositiveHits")>0);Assert.True(Count(cached,"layerChordNegativeHits")>0);
        Assert.True(StockCounters(cached)["ColumnQueries"]>0);
        Assert.True(StockCounters(cached)["CacheHits"]>0);
        Assert.True(Count(cached,"layerChordCoreRuns")<Count(literal,"layerChordCoreRuns"));
    }

    [Fact]
    public void DirectedBitKeysPreserveSignedZeroAndFifoAdmissionOrder()
    {
        var owner=Owner(2);var prove=Layer(owner);
        var a=new Point3D(-.3,+0d,-.2);var b=Good.B;var negative=a with{Y=BitConverter.Int64BitsToDouble(long.MinValue)};
        Assert.True(prove(a,b));Assert.True(prove(negative,b));Assert.Equal(2,Entries(owner));
        Assert.True(prove(a,b));Assert.Equal(1,Count(owner,"layerChordPositiveHits"));
        Assert.True(prove(b,a));Assert.Equal(1,Count(owner,"layerChordEvictions"));
        // A hit does not refresh FIFO: the first directed +0 entry was evicted.
        Assert.True(prove(a,b));Assert.Equal(2,Count(owner,"layerChordEvictions"));
        Assert.Equal(4,Count(owner,"layerChordCoreRuns"));Assert.Equal(2,Entries(owner));
    }

    [Theory]
    [InlineData(double.NaN)] [InlineData(double.PositiveInfinity)] [InlineData(double.NegativeInfinity)]
    public void NonfiniteQueriesRetainLiteralExceptionsAndPublishNothing(double value)
    {
        var owner=Owner(2);var original=Owner(0);var a=Good.A with{X=value};
        var expected=Record.Exception(()=>Layer(original)(a,Good.B));var actual=Record.Exception(()=>Layer(owner)(a,Good.B));
        Assert.NotNull(expected);Assert.Equal(expected.GetType(),actual?.GetType());
        Assert.Equal(0,Entries(owner));Assert.Equal(0,Count(owner,"layerChordPositiveHits"));
    }

    [Fact]
    public void PoseMemoAndPotentialShortWitnessCallbacksKeepTheirOriginalPath()
    {
        var log=new List<string>();var owner=Owner(2,log:log.Add);var prove=Layer(owner);
        Assert.True(prove(Good.A,Good.A));Assert.True(prove(Good.A,Good.A));
        Assert.Equal(0,Count(owner,"layerChordQueries"));Assert.Equal(1,Count(owner,"layerPoseCacheHits"));
        var b=Bad.A with{X=Bad.A.X+.00001};int calls=log.Count;
        Assert.False(prove(Bad.A,b));Assert.False(prove(Bad.A,b));
        Assert.Equal(2,Count(owner,"layerChordCoreRuns"));Assert.Equal(0,Count(owner,"layerChordNegativeHits"));
        Assert.Equal(0,Entries(owner));Assert.Equal(calls,log.Count);
    }

    [Fact]
    public void CanceledHitDoesNotReturnOrChangeExistingProofInventory()
    {
        using var cancellation=new CancellationTokenSource();var owner=Owner(2,cancellation.Token);var prove=Layer(owner);
        Assert.True(prove(Good.A,Good.B));cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>prove(Good.A,Good.B));
        Assert.Equal(1,Entries(owner));Assert.Equal(0,Count(owner,"layerChordPositiveHits"));
    }

    [Fact]
    public async Task CancellationAfterCoreAdmissionCannotPublishItsCompletedProof()
    {
        using var cancellation=new CancellationTokenSource();var owner=Owner(2,cancellation.Token);var prove=Layer(owner);
        var layers=Field<IList>(owner,"layers");var distance=layers[0]!.GetType().GetProperty("Distance")!.GetValue(layers[0])!;
        var queryLock=Field<object>(distance,"_queryLock");Task<bool>? work=null;
        Monitor.Enter(queryLock);
        try
        {
            work=Task.Run(()=>prove(Good.A,Good.B));
            Assert.True(SpinWait.SpinUntil(()=>Count(owner,"layerChordCoreRuns")==1,TimeSpan.FromSeconds(10)));
            cancellation.Cancel();
        }
        finally{Monitor.Exit(queryLock);}
        await Assert.ThrowsAnyAsync<OperationCanceledException>(async()=>await work!);
        Assert.Equal(0,Entries(owner));Assert.Equal(0,Count(owner,"layerChordPositivePublications"));
    }

    [Fact]
    public void WorkersOwnEmptyCachesAndCaptureOneNonadditiveQuiescentSourceSnapshot()
    {
        var owner=Owner(2);var prove=Layer(owner);Assert.True(prove(Good.A,Good.B));Assert.True(prove(Good.A,Good.B));
        var prior=VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value;
        try
        {
            VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value=0;
            var a=owner.CreateNativeQueryWorker(out _);var b=owner.CreateNativeQueryWorker(out _);
            Assert.Equal(2,Field<int>(a,"layerChordCacheEntries"));Assert.Equal(0,Entries(a));Assert.Equal(0,Entries(b));
            Assert.True(Layer(a)(Good.A,Good.B));Assert.Equal(1,Entries(a));Assert.Equal(0,Entries(b));Assert.Equal(1,Entries(owner));
            Assert.Equal(1,Count(owner,"layerChordPositiveHits"));Assert.Equal(0,Count(b,"layerChordQueries"));
            Assert.Contains("sourceLayerChordPositiveHits=1",a.ProfileDepthDiagnostics);
            Assert.Contains("sourceLayerChordScope=quiescent-startup-snapshot-repeated-per-worker-not-additive",a.ProfileDepthDiagnostics);
            Assert.Contains("layerChordCoreElapsedScope=summed-invocation-elapsed-not-CPU-or-wall",a.ProfileDepthDiagnostics);
        }
        finally{VCarveMaterialSurface.LayerChordCacheEntriesOverride.Value=prior;}
    }

    [Theory]
    [InlineData(-1)] [InlineData(32769)]
    public void InvalidCapacitiesRefuseBeforeGeometryConstruction(int capacity)
        =>Assert.Throws<ArgumentOutOfRangeException>(()=>Owner(capacity));

    [Fact]
    public void NativePlansCommandsSourceAndOpaqueCallbackOrderMatchLiteralOwner()
    {
        Point3D[] source=[new(-.5,0,-.2),new(-.25,.03,-.2),new(0,.05,-.2),new(.25,.03,-.2),new(.5,0,-.2)];
        var before=source.ToArray();var path=new TaggedToolpath(source,[new(0,4,ToolpathSpanKind.Cut)],0,"layer-cache",0,0,detectCurves:false);
        IReadOnlyList<VCarveNativeCurvePlan> Run(VCarveMaterialSurface owner,List<string> callbacks)
            =>VCarveNativeCurvePlanning.Nominate([path],[[Box(0,0,2)]],Cutter,0,settings:new(.03,.1),
                stockChord:(a,b)=>{callbacks.Add($"chord:{a}:{b}");return owner.IsLegalNativeChord(a,b);},
                stockCurve:m=>{callbacks.Add($"curve:{m}");return owner.IsLegalNativeCurve(m);});
        var literal=Owner(0);var cached=Owner(32768);var expectedCalls=new List<string>();var actualCalls=new List<string>();
        var expected=Run(literal,expectedCalls);var actual=Run(cached,actualCalls);
        Assert.Equal(expected.SelectMany(p=>p.Primitives),actual.SelectMany(p=>p.Primitives));Assert.Equal(expectedCalls,actualCalls);
        Assert.All(actual,p=>Assert.True(p.CandidateVerified,p.Status));Assert.Equal(before,source);
        Point3D Posted(Point3D p)=>new(VCarveCommandRounding.Round((float)p.X),VCarveCommandRounding.Round((float)p.Y),VCarveCommandRounding.Round((float)p.Z));
        foreach(var primitive in actual.SelectMany(p=>p.Primitives))
        {
            var m=primitive.Move;Assert.True(cached.IsLegalNativeCurve(m));
            var command=m.Kind==ToolMoveKind.CubicBezier
                ?ToolMove.CubicBezier(Posted(m.Start),Posted(m.Control1),Posted(m.Control2),Posted(m.End),m.MaxError)
                :ToolMove.Line(Posted(m.Start),Posted(m.End));
            Assert.True(cached.IsLegalNativeCurve(command));
        }
    }
}
