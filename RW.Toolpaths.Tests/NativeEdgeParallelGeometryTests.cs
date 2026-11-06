using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeEdgeParallelGeometryTests
{
    private readonly record struct StockCall(bool Curve,Point3D A,Point3D B,ToolMove Move);
    private sealed record RunResult(VCarveNativeFit? Fit,VCarveNativeEdgeAudit Audit,StockCall[] Calls);
    private static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
    private static readonly PointD[][] Boundary=[[new(-2,-2),new(11,-2),new(11,2),new(-2,2)]];
    private static Point3D[] Source(int count=97)
    {
        var curve=ToolMove.CubicBezier(new(0,0,-.5),new(3,1,-.5),new(6,1,-.5),new(9,0,-.5),0);
        return Enumerable.Range(0,count).Select(i=>VCarveNativeCurveMath.At(curve,i/(double)(count-1))).ToArray();
    }
    private static RunResult Run(Point3D[] points,bool parallel,VCarveNativeFitSettings? settings=null,
        IReadOnlyList<double>? scales=null,VCarveNativeFreeJoinEndpoints? freeJoin=null,
        Func<Point3D,Point3D,bool>? chord=null,Func<ToolMove,bool>? curve=null,VCarveCutterProfile? cutter=null,
        IReadOnlyList<IReadOnlyList<PointD>>? boundary=null)
    {
        int callingThread=Environment.CurrentManagedThreadId,curveAttempts=0;
        var calls=new List<StockCall>();
        var guard=new VCarveNativeContainment(boundary??Boundary,cutter??Cutter,
            stockChord:(a,b)=>{
                Assert.Equal(callingThread,Environment.CurrentManagedThreadId);
                calls.Add(new(false,a,b,default));return chord?.Invoke(a,b)??true;
            },stockCurve:move=>{
                Assert.Equal(callingThread,Environment.CurrentManagedThreadId);
                calls.Add(new(true,default,default,move));return curve?.Invoke(move)??++curveAttempts>=3;
            });
        double length=0;for(int i=1;i<points.Length;i++)length+=VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[1],points[0]));
        var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[^2]));
        var fit=VCarveNativeEdgeSearch.Find(points,0,points.Length-1,u,v,length,guard,cutter??Cutter,out var audit,
            settings??new(1,.3),scales,freeJoin,parallelGeometryProofs:parallel);
        return new(fit,audit,calls.ToArray());
    }
    private static void Exact(RunResult serial,RunResult parallel)
    {
        Assert.Equal(serial.Fit,parallel.Fit);Assert.Equal(serial.Calls,parallel.Calls);
        int[] Counts(VCarveNativeEdgeAudit a)=>[a.DepthRejected,a.OrderRejected,a.FitRejected,a.PoseRejected,
            a.LengthRejected,a.ContinuousRejected,a.ProofBudget,a.Qualified,a.LowerBoundRejected,a.GeometryProofs,a.PlanarProgressRejected];
        Assert.Equal(Counts(serial.Audit),Counts(parallel.Audit));Assert.Equal(serial.Audit.Outcome,parallel.Audit.Outcome);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PureParallelCertificatesPreserveTheExactFitAuditAndOrderedStockCallbacks(bool freeJoin)
    {
        var points=Source();
        var freedom=freeJoin?new VCarveNativeFreeJoinEndpoints(points[0] with{Y=.01},points[^1] with{Y=.01},-.5,-.5):null;
        var serial=Run(points,false,freeJoin:freedom);var parallel=Run(points,true,freeJoin:freedom);
        Exact(serial,parallel);Assert.NotNull(serial.Fit);
        Assert.True(serial.Calls.Count(c=>c.Curve)>=3);Assert.Equal(1,serial.Audit.GeometryWorkers);
        if(GeometryParallelism.WorkerLimit>1)
        {
            Assert.True(parallel.Audit.BufferedGeometryCandidates>=32);
            Assert.InRange(parallel.Audit.GeometryWorkers,2,GeometryParallelism.WorkerLimit);
        }
    }

    [Fact]
    public void ExtendedHandleInventoryKeepsEveryNominationAcrossBoundedProofBatches()
    {
        double[] scales=[1,.85,1.15,.7,1.3,.5,.25,.1,.03,.01,.003,2,3,4];
        var points=Source();var serial=Run(points,false,new(100,.3),scales);
        var parallel=Run(points,true,new(100,.3),scales);
        Exact(serial,parallel);
        if(GeometryParallelism.WorkerLimit>1)Assert.True(parallel.Audit.BufferedGeometryCandidates>242);
    }

    [Fact]
    public void SmallSourcesAndNestedGeometryWorkersRetainSerialProofs()
    {
        var small=Run(Source(63),true);
        Assert.Equal(1,small.Audit.GeometryWorkers);Assert.Equal(0,small.Audit.BufferedGeometryCandidates);
        using var scope=GeometryParallelism.EnterWorker();
        var nested=Run(Source(),true);
        Assert.Equal(1,nested.Audit.GeometryWorkers);Assert.Equal(0,nested.Audit.BufferedGeometryCandidates);
    }

    [Fact]
    public void AuthenticStockContactRetainsTheExactPhysicalProofQuerySequence()
    {
        var assembly=typeof(NativeEdgeParallelGeometryTests).Assembly;
        using var stream=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("stock-contact-branch.json.gz")))!;
        using var gzip=new GZipStream(stream,CompressionMode.Decompress);using var document=JsonDocument.Parse(gzip);
        var data=document.RootElement;var json=new JsonSerializerOptions{IncludeFields=true};
        T Read<T>(JsonElement element)=>element.Deserialize<T>(json)!;
        VCarveCutterProfile Profile(JsonElement element)=>new(Read<CarvingCutterSection[]>(element.GetProperty("Sections")),element.GetProperty("MaximumDepth").GetDouble());
        double surface=data.GetProperty("SurfaceZ").GetDouble(),maximum=data.GetProperty("MaximumDepth").GetDouble();
        var profile=new VCarveCutterProfile(Read<CarvingCutterSection[]>(data.GetProperty("Profile")),maximum);
        var boundaries=Read<PointD[][][]>(data.GetProperty("Boundaries"));var saved=data.GetProperty("Stock");
        var stock=new VCarveClearedStock(Read<VCarveRoughingChord[]>(saved.GetProperty("Chords")),Read<double[]>(saved.GetProperty("FloorDepths")),Read<PointD[]>(saved.GetProperty("PartBoundary")),
            saved.GetProperty("ProfileSweeps").EnumerateArray().Select(s=>new VCarveClearedSweep(Profile(s.GetProperty("Profile")),Read<VCarveRoughingChord[]>(s.GetProperty("Chords")))).ToArray());
        var raw=Read<Point3D[]>(data.GetProperty("Paths")[0]).Select(p=>p with{Z=p.Z-surface}).ToArray();
        // Exact source chords are subdivided only to exercise the parallel
        // threshold. Both runs receive this same immutable point array.
        var points=Enumerable.Range(0,raw.Length-1).SelectMany(i=>Enumerable.Range(0,8)
            .Select(k=>VCarveNativeCurveMath.Lerp(raw[i],raw[i+1],k/8d))).Append(raw[^1]).ToArray();
        RunResult Physical(bool parallel)
        {
            var owner=new VCarveMaterialSurface(boundaries.SelectMany(b=>b),stock,profile,maximum,surface,default,null);
            return Run(points,parallel,new(.3,.3),chord:owner.IsLegalNativeChord,curve:owner.IsLegalNativeCurve,cutter:profile,boundary:boundaries[11]);
        }
        Exact(Physical(false),Physical(true));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationAfterPureProofsStillThrowsOnTheCallingStockThread(bool parallel)
    {
        using var cancellation=new CancellationTokenSource();int callingThread=Environment.CurrentManagedThreadId;bool stockReached=false;
        var guard=new VCarveNativeContainment(Boundary,Cutter,cancellation.Token,stockChord:(_,_)=>true,stockCurve:_=>{
            Assert.Equal(callingThread,Environment.CurrentManagedThreadId);stockReached=true;cancellation.Cancel();
            cancellation.Token.ThrowIfCancellationRequested();return true;
        });
        var points=Source();double length=0;for(int i=1;i<points.Length;i++)length+=VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[1],points[0]));
        var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[^2]));
        Assert.Throws<OperationCanceledException>(()=>VCarveNativeEdgeSearch.Find(points,0,points.Length-1,u,v,length,guard,Cutter,out _,new(1,.3),parallelGeometryProofs:parallel));
        Assert.True(stockReached);
    }
}
